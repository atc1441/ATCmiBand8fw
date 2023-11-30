//*****************************************************************************
//
//! @file am_devices_spipsram.h
//!
//! @brief General SPI PSRAM driver.
//!
//! @addtogroup spipsram SPI PSRAM Driver
//! @ingroup devices
//! @{
//
//**************************************************************************

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

#ifndef AM_DEVICES_SPIPSRAM_H
#define AM_DEVICES_SPIPSRAM_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name  Global definitions for psram commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIPSRAM_WRITE             0x02
#define AM_DEVICES_SPIPSRAM_READ              0x03
#define AM_DEVICES_SPIPSRAM_FAST_READ         0x0B
#define AM_DEVICES_SPIPSRAM_QUAD_MODE_ENTER   0x35
#define AM_DEVICES_SPIPSRAM_QUAD_WRITE        0x38
#define AM_DEVICES_SPIPSRAM_RESET_ENABLE      0x66
#define AM_DEVICES_SPIPSRAM_RESET_MEMORY      0x99
#define AM_DEVICES_SPIPSRAM_READ_ID           0x9F
#define AM_DEVICES_SPIPSRAM_HALF_SLEEP_ENTER  0xC0
#define AM_DEVICES_SPIPSRAM_QUAD_READ         0xEB
#define AM_DEVICES_SPIPSRAM_QUAD_MODE_EXIT    0xF5
//! @}

//*****************************************************************************
//
//! @name Device specific identification.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIPSRAM_KGD_PASS          0x5D0D
#define AM_DEVICES_SPIPSRAM_KGD_FAIL          0x550D

// Page size - limits the bust write/read
#define AM_DEVICES_SPIPSRAM_PAGE_SIZE         1024

//! @}

extern uint32_t g_APS6404LCS;

//*****************************************************************************
//
//! @name According to APS6404L tCEM restriction, we define maximum bytes for each speed empirically
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIPSRAM_48MHZ_MAX_BYTES   32
#define AM_DEVICES_SPIPSRAM_24MHZ_MAX_BYTES   16
#define AM_DEVICES_SPIPSRAM_16MHZ_MAX_BYTES   10
#define AM_DEVICES_SPIPSRAM_12MHZ_MAX_BYTES   6
#define AM_DEVICES_SPIPSRAM_8MHZ_MAX_BYTES    3
//! @}

#define AM_DEVICES_APS6404L_MAX_DEVICE_NUM    8

//*****************************************************************************
//
//! @name  Global type definitions.
//! @{
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_SPIPSRAM_STATUS_SUCCESS,
    AM_DEVICES_SPIPSRAM_STATUS_ERROR
} am_devices_spipsram_status_t;

typedef struct
{
    uint32_t ui32ClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ChipSelectNum ;
} am_devices_spipsram_config_t;
//! @}

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the spipsram driver.
//!
//! @param ui32Module     - IOM Module#
//! @param pDevConfig
//! @param ppHandle
//! @param ppIomHandle
//!
//! @note This function should be called before any other am_devices_spipsram
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_init(uint32_t ui32Module,
                                         am_devices_spipsram_config_t *pDevConfig,
                                         void **ppHandle,
                                         void **ppIomHandle);

//*****************************************************************************
//
//! @brief
//! @param ui32Module
//! @param pDevConfig
//! @param ppHandle
//! @param ppIomHandle
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_init_no_check(uint32_t ui32Module,
                                                  am_devices_spipsram_config_t *pDevConfig,
                                                  void **ppHandle,
                                                  void **ppIomHandle);


//*****************************************************************************
//
//! @brief DeInitialize the spipsram driver.
//!
//! @param pHandle     - Pointer to device handle
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_term(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pHandle   - Pointer to driver handle
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_read_id(void *pHandle, uint32_t *pDeviceID);

//*****************************************************************************
//
//! @brief Reset the external psram
//!
//! @param pHandle
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer     - Buffer to store the received data from the psram
//! @param ui32ReadAddress  - Address of desired data in external psram
//! @param ui32NumBytes     - Number of bytes to read from external psram
//! @param bWaitForCompletion - Blocks when true
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_read(void *pHandle,
                                         uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer     - Buffer to store the received data from the psram
//! @param ui32ReadAddress  - Address of desired data in external psram
//! @param ui32NumBytes     - Number of bytes to read from external psram
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_read_adv(void *pHandle,
                                           uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           uint32_t ui32PauseCondition,
                                           uint32_t ui32StatusSetClr,
                                           am_hal_iom_callback_t pfnCallback,
                                           void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
//! @param bWaitForCompletion
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external psram at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target psram
//! memory or underflow the pui8TxBuffer array
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_write(void *pHandle,  // dox check XXX
                                            uint8_t *pui8TxBuffer,
                                            uint32_t ui32WriteAddress,
                                            uint32_t ui32NumBytes,
                                            bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param puiTxBuffer
//! @param ui32WriteAddress
//! @param ui32NumBytes
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_write_adv(void *pHandle,
                                               uint8_t *puiTxBuffer,
                                               uint32_t ui32WriteAddress,
                                               uint32_t ui32NumBytes,
                                               uint32_t ui32PauseCondition,
                                               uint32_t ui32StatusSetClr,
                                               am_hal_iom_callback_t pfnCallback,
                                               void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer     - Buffer to store the received data from the psram
//! @param ui32ReadAddress  - Address of desired data in external psram
//! @param ui32NumBytes     - Number of bytes to read from external psram
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! This function reads the external psram at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_nonblocking_read(void *pHandle,
                                                       uint8_t *pui8RxBuffer,
                                                       uint32_t ui32ReadAddress,
                                                       uint32_t ui32NumBytes,
                                                       am_hal_iom_callback_t pfnCallback,
                                                       void *pCallbackCtxt);

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
extern uint32_t am_devices_spipsram_nonblocking_write(void *pHandle,
                                                      uint8_t *pui8TxBuffer,
                                                      uint32_t ui32WriteAddress,
                                                      uint32_t ui32NumBytes,
                                                      am_hal_iom_callback_t pfnCallback,
                                                      void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8TxBuffer     - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes     - Number of bytes to write to the external psram
//!
//! @note This function uses the data in the provided pui8TxBuffer and copies it to
//! the external psram at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target psram
//! memory or underflow the pui8TxBuffer array
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_blocking_write(void *pHandle,
                                                   uint8_t *pui8TxBuffer,
                                                   uint32_t ui32WriteAddress,
                                                   uint32_t ui32NumBytes);

//*****************************************************************************
//
//! @brief Reads the contents of the fram into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer     - Buffer to store the received data from the flash
//! @param ui32ReadAddress  - Address of desired data in external flash
//! @param ui32NumBytes     - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_blocking_read(void *pHandle,
                                                  uint8_t *pui8RxBuffer,
                                                  uint32_t ui32ReadAddress,
                                                  uint32_t ui32NumBytes);

//*****************************************************************************
//
//! @brief Generic Command Write function.
//!
//! @param pHandle        - pointer to driver handle
//! @param bHiPrio        - make this transfer high priority
//! @param ui32InstrLen   - number of offset bytes to send
//! @param ui64Instr      - offset bytes that are sent
//! @param pData          - point to array holding tx data
//! @param ui32NumBytes   - number of bytes to tx
//! @param bContinue      - keep chipselect asserted after transfer
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_command_write(void *pHandle,
                          bool bHiPrio,
                          uint32_t ui32InstrLen,
                          uint64_t ui64Instr,
                          uint32_t *pData,
                          uint32_t ui32NumBytes,
                          bool bContinue);

//*****************************************************************************
//
//! @brief
//! @param pHandle
//! @param bHiPrio
//! @param ui32InstrLen
//! @param ui64Instr
//! @param pData
//! @param ui32NumBytes
//! @param bContinue
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_spipsram_command_read(
                        void *pHandle,
                        bool bHiPrio,
                        uint32_t ui32InstrLen,
                        uint64_t ui64Instr,
                        uint32_t *pData,
                        uint32_t ui32NumBytes,
                        bool bContinue);


#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_SPIPSRAM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

