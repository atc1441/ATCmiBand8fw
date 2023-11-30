//*****************************************************************************
//
//! @file am_devices_mspi_psram_aps6404l.h
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

#ifndef AM_DEVICES_MSPI_PSRAM_APS6404L_H
#define AM_DEVICES_MSPI_PSRAM_APS6404L_H

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
#define AM_DEVICES_MSPI_PSRAM_WRITE             0x02
#define AM_DEVICES_MSPI_PSRAM_READ              0x03
#define AM_DEVICES_MSPI_PSRAM_FAST_READ         0x0B
#define AM_DEVICES_MSPI_PSRAM_QUAD_MODE_ENTER   0x35
#define AM_DEVICES_MSPI_PSRAM_QUAD_WRITE        0x38
#define AM_DEVICES_MSPI_PSRAM_RESET_ENABLE      0x66
#define AM_DEVICES_MSPI_PSRAM_RESET_MEMORY      0x99
#define AM_DEVICES_MSPI_PSRAM_READ_ID           0x9F
#define AM_DEVICES_MSPI_PSRAM_APS6404L_HALF_SLEEP_ENTER  0xC0
#define AM_DEVICES_MSPI_PSRAM_QUAD_READ         0xEB
#define AM_DEVICES_MSPI_PSRAM_QUAD_MODE_EXIT    0xF5
//! @}

//*****************************************************************************
//
//! @note The following definitions are typically specific to a multibit spi psram device.
//! They should be tailored
//
//*****************************************************************************
//
//! @name  Device specific identification.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_KGD_PASS          0x5D0D
#define AM_DEVICES_MSPI_PSRAM_KGD_FAIL          0x550D
//! @}

//! @name Page size - limits the bust write/read
//! @{

#define AM_DEVICES_MSPI_PSRAM_PAGE_SIZE         1024
//#define AM_DEVICES_MSPI_PSRAM_TEST_BLOCK_SIZE   64*1024
#define AM_DEVICES_MSPI_PSRAM_TEST_BLOCK_SIZE   8*1024
//! @}

//! @{
//! According to APS6404L tCEM restriction, we define maximum bytes for each speed empirically
#define AM_DEVICES_MSPI_PSRAM_48MHZ_MAX_BYTES   128
#define AM_DEVICES_MSPI_PSRAM_24MHZ_MAX_BYTES   64
#define AM_DEVICES_MSPI_PSRAM_16MHZ_MAX_BYTES   32
#define AM_DEVICES_MSPI_PSRAM_12MHZ_MAX_BYTES   16
#define AM_DEVICES_MSPI_PSRAM_8MHZ_MAX_BYTES    8

//! @}

//*****************************************************************************
//
//! Global definitions for the MSPI instance to use.
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM    2

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS,
    AM_DEVICES_MSPI_PSRAM_STATUS_ERROR
} am_devices_mspi_psram_status_t;

typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ScramblingStartAddr;
    uint32_t ui32ScramblingEndAddr;
} am_devices_mspi_psram_config_t;

typedef struct
{
    uint32_t ui32Turnaround;
    uint32_t ui32Rxneg;
    uint32_t ui32Rxdqsdelay;
} am_devices_mspi_psram_sdr_timing_config_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the mspi_psram driver.
//!
//! @param ui32Module           - MSPI module
//! @param pDevCfg              - CFG struct for the device
//! @param ppHandle             - Driver handler, is returned
//! @param ppMspiHandle         - MSPI handler which needs to be returned
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_init(uint32_t ui32Module,
                                                   am_devices_mspi_psram_config_t *pDevCfg,
                                                   void **ppHandle,
                                                   void **ppMspiHandle);

//*****************************************************************************
//
//! @brief DeInitialize the mspi_psram driver.
//!
//! @param pHandle - Pointer to driver handle
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_deinit(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_id(void *pHandle);

//*****************************************************************************
//
//! @brief Reset the external psram
//!
//! @param pHandle - Pointer to driver handle
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer     - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress  - Address of desired data in external PSRAM
//! @param ui32NumBytes     - Number of bytes to read from external PSRAM
//! @param bWaitForCompletion - Wait for transaction completion before exiting
//!
//! This function reads the external PSRAM at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.  If the bWaitForCompletion is true,
//! then the function will poll for DMA completion indication flag before
//! returning.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_read(void *pHandle,
                                           uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param pHandle              - Pointer to driver handle
//! @param pui8RxBuffer         - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress      - Address of desired data in external PSRAM
//! @param ui32NumBytes         - Number of bytes to read from external PSRAM
//! @param ui32PauseCondition   - Pause condition before transaction is executed
//! @param ui32StatusSetClr     - Post-transaction CQ condition
//! @param pfnCallback          - Post-transaction callback function
//! @param pCallbackCtxt        - Post-transaction callback context
//!
//! This function reads the external PSRAM at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.  The Command Queue pre and post
//! transaction conditions and a callback function and context are also
//! provided.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_read_adv(void *pHandle,
                                               uint8_t *pui8RxBuffer,
                                               uint32_t ui32ReadAddress,
                                               uint32_t ui32NumBytes,
                                               uint32_t ui32PauseCondition,
                                               uint32_t ui32StatusSetClr,
                                               am_hal_mspi_callback_t pfnCallback,
                                               void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8TxBuffer     - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes     - Number of bytes to write to the external psram
//! @param bWaitForCompletion - When true function call will block
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
extern uint32_t am_devices_mspi_psram_write(void *pHandle,
                                            uint8_t *pui8TxBuffer,
                                            uint32_t ui32WriteAddress,
                                            uint32_t ui32NumBytes,
                                            bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle          - Pointer to driver handle
//! @param puiTxBuffer      - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes     - Number of bytes to write to the external psram
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
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
extern uint32_t am_devices_mspi_psram_write_adv(void *pHandle,
                                                uint8_t *puiTxBuffer,
                                                uint32_t ui32WriteAddress,
                                                uint32_t ui32NumBytes,
                                                uint32_t ui32PauseCondition,
                                                uint32_t ui32StatusSetClr,
                                                am_hal_mspi_callback_t pfnCallback,
                                                void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into XIP mode.
//!
//! @param pHandle  - Pointer to driver handle
//!
//! This function sets the external psram device and the MSPI into XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_enable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from XIP mode.
//!
//! @param pHandle  - Pointer to driver handle
//!
//! This function removes the external device and the MSPI from XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_disable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into scrambling mode.
//!
//! @param pHandle  - Pointer to driver handle
//!
//! This function sets the external psram device and the MSPI into scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_enable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from scrambling mode.
//!
//! @param pHandle  - Pointer to driver handle
//!
//! This function removes the external device and the MSPI from scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_disable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pHandle          - Pointer to driver handle
//! @param pui8RxBuffer - Buffer to store the received data from the psram
//! @param ui32ReadAddress - Address of desired data in external psram
//! @param ui32NumBytes - Number of bytes to read from external psram
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
extern uint32_t am_devices_mspi_psram_read_hiprio(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);
//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param pui8RxBuffer
//! @param ui32ReadAddress
//! @param ui32NumBytes
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_nonblocking_read(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle      - Pointer to driver handle
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
//! @param pfnCallback
//! @param pCallbackCtxt
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
extern uint32_t am_devices_mspi_psram_write_hiprio(void *pHandle, uint8_t *pui8TxBuffer,
                           uint32_t ui32WriteAddress,
                           uint32_t ui32NumBytes,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
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
extern uint32_t am_devices_mspi_psram_nonblocking_write(void *pHandle, uint8_t *pui8TxBuffer,
                           uint32_t ui32WriteAddress,
                           uint32_t ui32NumBytes,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Checks PSRAM timing and determine a delay setting.
//!
//! @param module
//! @param pDevCfg
//! @param pDevSdrCfg
//!
//! This function scans through the delay settings of MSPI DDR mode and selects
//! the best parameter to use by tuning TURNAROUND/RXNEG/RXDQSDELAY0 values.
//! This function is only valid in DDR mode and ENABLEDQS0 = 0.
//!
//! @return 32-bit status, scan result in structure type
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_sdr_init_timing_check(uint32_t module,
                                            am_devices_mspi_psram_config_t *pDevCfg,
                                            am_devices_mspi_psram_sdr_timing_config_t *pDevSdrCfg);


//*****************************************************************************
//
//! @brief Apply given SDR timing settings to target MSPI instance.
//!
//! @param pHandle - Handle to the PSRAM.
//! @param pDevSdrCfg - Pointer to the ddr timing config structure
//!
//! This function applies the ddr timing settings to the selected mspi instance.
//! This function must be called after MSPI instance is initialized into
//! ENABLEFINEDELAY0 = 1 mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_apply_sdr_timing(void *pHandle,
                                        am_devices_mspi_psram_sdr_timing_config_t *pDevSdrCfg);
#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_PSRAM_APS6404L_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

