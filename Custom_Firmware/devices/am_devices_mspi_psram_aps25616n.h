//*****************************************************************************
//
//! @file am_devices_mspi_psram_aps25616n.h
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

#ifndef AM_DEVICES_MSPI_PSRAM_APS25616N_H
#define AM_DEVICES_MSPI_PSRAM_APS25616N_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Global definitions for psram commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_DDR_GLOBAL_RESET 0xFFFF
#define AM_DEVICES_MSPI_PSRAM_DDR_READ        0x2020
#define AM_DEVICES_MSPI_PSRAM_DDR_WRITE       0xA0A0
#define AM_DEVICES_MSPI_PSRAM_DDR_READ_REGISTER 0x4040
#define AM_DEVICES_MSPI_PSRAM_DDR_WRITE_REGISTER 0xC0C0

//! @}

//*****************************************************************************
//
//! @name Global definitions for psram
//! @{
//!
//! @note The following definitions are typically specific to a multibit spi psram device.
//! They should be tailored
//
//*****************************************************************************
//
//! @name Device specific identification.
//! @{
//
//*****************************************************************************
// Page size - limits the bust write/read
#define AM_DEVICES_MSPI_PSRAM_PAGE_SIZE         1024
#define AM_DEVICES_MSPI_PSRAM_TEST_BLOCK_SIZE   8*1024
//! @}

//*****************************************************************************
//
//! @name Global definitions for the MSPI instance to use.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_PSRAM_MAX_DEVICE_NUM    2
//! @}
//! @}

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

typedef enum
{
    AM_DEVICES_MSPI_PSRAM_APS25616n_RLC_3,
    AM_DEVICES_MSPI_PSRAM_APS25616n_RLC_4,
    AM_DEVICES_MSPI_PSRAM_APS25616n_RLC_5,  //device power up default
    AM_DEVICES_MSPI_PSRAM_APS25616n_RLC_6,
    AM_DEVICES_MSPI_PSRAM_APS25616n_RLC_7,
}am_devices_mspi_psram_aps25616n_rlc_e;

typedef enum
{
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_3,
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_7,
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_5,  //device power up default
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_RSV3,
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_4,
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_RSV5,
    AM_DEVICES_MSPI_PSRAM_APS25616n_WLC_6,
}am_devices_mspi_psram_aps25616n_wlc_e;

typedef struct
{
    uint8_t     ui8VendorId;
    uint8_t     ui8DeviceId;
    uint32_t    ui32BaseAddr;
    uint32_t    ui32DeviceSizeKb;
} am_devices_mspi_psram_info_t;

typedef struct
{
    am_hal_mspi_device_e                    eDeviceConfig;
    am_hal_mspi_clock_e                     eClockFreq;
    uint32_t                                *pNBTxnBuf;
    uint32_t                                ui32NBTxnBufLength;
    uint32_t                                ui32ScramblingStartAddr;
    uint32_t                                ui32ScramblingEndAddr;
} am_devices_mspi_psram_config_t;

typedef struct
{
    uint32_t ui32Turnaround;
    uint32_t ui32Rxneg;
    uint32_t ui32Rxdqsdelay;
    uint32_t ui32Txdqsdelay;
} am_devices_mspi_psram_ddr_timing_config_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Initialize the mspi_psram driver.
//!
//! @param ui32Module - MSPI instance
//! @param pDevCfg - MSPI device structure describing the target spi psram.
//! @param ppHandle - Device handle which needs to be return
//! @param ppMspiHandle - MSPI handle which needs to be return
//!
//! This function should be called before any other am_devices_mspi_psram
//! functions. It is used to set tell the other functions how to communicate
//! with the external psram hardware.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_init(uint32_t ui32Module,
                                       am_devices_mspi_psram_config_t *pDevCfg,
                                       void **ppHandle,
                                       void **ppMspiHandle);

//*****************************************************************************
//
//! @brief DeInitialize the mspi_psram driver.
//!
//! @param pHandle - MSPI Device handle.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_deinit(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param pHandle - MSPI Device handle.
//! @param pui8RxBuffer - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress - Address of desired data in external PSRAM
//! @param ui32NumBytes - Number of bytes to read from external PSRAM
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_read(void *pHandle,
                                         uint8_t *pui8RxBuffer,
                                         uint32_t ui32ReadAddress,
                                         uint32_t ui32NumBytes,
                                         bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Reads the contents of the external PSRAM into a buffer.
//!
//! @param pHandle - MSPI Device handle.
//! @param pui8RxBuffer - Buffer to store the received data from the PSRAM
//! @param ui32ReadAddress - Address of desired data in external PSRAM
//! @param ui32NumBytes - Number of bytes to read from external PSRAM
//! @param ui32PauseCondition - Pause condition before transaction is executed
//! @param ui32StatusSetClr - Post-transaction CQ condition
//! @param pfnCallback - Post-transaction callback function
//! @param pCallbackCtxt - Post-transaction callback context
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_read_adv(void *pHandle,
                                             uint8_t *pui8RxBuffer,
                                             uint32_t ui32ReadAddress,
                                             uint32_t ui32NumBytes,
                                             uint32_t ui32PauseCondition,
                                             uint32_t ui32StatusSetClr,
                                             am_hal_mspi_callback_t pfnCallback,
                                             void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer
//!        with high priority
//!
//! @param pHandle - MSPI Device handle.
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_read_hiprio(void *pHandle,
                                            uint8_t *pui8RxBuffer,
                                            uint32_t ui32ReadAddress,
                                            uint32_t ui32NumBytes,
                                            am_hal_mspi_callback_t pfnCallback,
                                            void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Reads the contents of the external psram into a buffer.
//!
//! @param pHandle - MSPI Device handle.
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_nonblocking_read(
                                             void *pHandle,
                                             uint8_t *pui8RxBuffer,
                                             uint32_t ui32ReadAddress,
                                             uint32_t ui32NumBytes,
                                             am_hal_mspi_callback_t pfnCallback,
                                             void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle - Device handle of the external psram
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_write(void *pHandle,
                                          uint8_t *pui8TxBuffer,
                                          uint32_t ui32WriteAddress,
                                          uint32_t ui32NumBytes,
                                          bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Programs the given range of psram addresses.
//!
//! @param pHandle - Device handle of the external psram
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_write_adv(void *pHandle,
                                          uint8_t *puiTxBuffer,
                                          uint32_t ui32WriteAddress,
                                          uint32_t ui32NumBytes,
                                          uint32_t ui32PauseCondition,
                                          uint32_t ui32StatusSetClr,
                                          am_hal_mspi_callback_t pfnCallback,
                                          void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the contents of the external psram into a buffer
//!        with high priority
//!
//! @param pHandle - Device handle of the external psram
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_write_hiprio(void *pHandle,
                                             uint8_t *pui8TxBuffer,
                                             uint32_t ui32WriteAddress,
                                             uint32_t ui32NumBytes,
                                             am_hal_mspi_callback_t pfnCallback,
                                             void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Programs the contents of the external psram into a buffer
//!
//! @param pHandle - Device handle of the external psram
//! @param ui32DeviceNumber - Device number of the external psram
//! @param pui8TxBuffer - Buffer to write the external psram data from
//! @param ui32WriteAddress - Address to write to in the external psram
//! @param ui32NumBytes - Number of bytes to write to the external psram
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
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_nonblocking_write(void *pHandle,
                                                      uint8_t *pui8TxBuffer,
                                                      uint32_t ui32WriteAddress,
                                                      uint32_t ui32NumBytes,
                                                      am_hal_mspi_callback_t pfnCallback,
                                                      void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into XIP mode.
//!
//! @param pHandle - Device handle of the external psram
//!
//! This function sets the external psram device and the MSPI into XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_enable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from XIP mode.
//!
//! @param pHandle - Device handle of the external psram
//!
//! This function removes the external device and the MSPI from XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_disable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external psram into scrambling mode.
//!
//! @param pHandle - Device handle of the external psram
//!
//! This function sets the external psram device and the MSPI into scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_enable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external psram from scrambling mode.
//!
//! @param pHandle - Device handle of the external psram
//!
//! This function removes the external device and the MSPI from scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_disable_scrambling(void *pHandle);

//*****************************************************************************
//! @brief Reset the external psram.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function resets the external psram
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the ID of the external psram and returns the value.
//!
//! @param pHandle  - Pointer to the psram device handle
//!
//! This function reads the device ID register of the external psram, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status or 16-bit ID
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_id(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the information of the external psram and returns the value.
//!
//! @param pHandle  - Pointer to the psram device handle
//! @param pPsramInfo - Pointer to the psram info struct
//!
//! This function reads the device information registers of the external psram, and returns
//! the result in am_devices_mspi_psram_info_t
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_info(void *pHandle, am_devices_mspi_psram_info_t *pPsramInfo);

//*****************************************************************************
//
//! @brief Checks PSRAM timing and determine a delay setting.
//!
//! @param module - MSPI Module Number.
//! @param pDevCfg - Pointer to the psram device config structure
//! @param pDevDdrCfg - Pointer to the ddr timing config structure
//!
//! This function scans through the delay settings of MSPI DDR mode and selects
//! the best parameter to use by tuning TURNAROUND/RXNEG/RXDQSDELAY0 values.
//! This function is only valid in DDR mode and ENABLEDQS0 = 0.
//!
//! @return 32-bit status, scan result in structure type
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_ddr_init_timing_check(uint32_t module,
                        am_devices_mspi_psram_config_t *pDevCfg,
                        am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg);

//*****************************************************************************
//
//! @brief Apply given DDR timing settings to target MSPI instance.
//!
//! @param pHandle - Handle to the PSRAM.
//! @param pDevDdrCfg - Pointer to the ddr timing config structure
//!
//! This function applies the ddr timing settings to the selected mspi instance.
//! This function must be called after MSPI instance is initialized into
//! ENABLEFINEDELAY0 = 1 mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_apply_ddr_timing(void *pHandle,
                       am_devices_mspi_psram_ddr_timing_config_t *pDevDdrCfg);

//*****************************************************************************
//
//! @brief Verify the External Chip Configuration matches Device settings
//!
//! @param pHandle - Handle to the PSRAM.
//! @param ui8RLC  - Read Latency Code
//! @param ui8WLC  - Write LAtency Code
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_verify_config(void *pHandle,
                                                                uint8_t ui8RLC,
                                                                uint8_t ui8WLC);

//*****************************************************************************
//
//! @brief Function to Enter Half Sleep
//!
//! @param pHandle - Handle to the PSRAM.
//!
//! Send a command to Enter Half Sleep Mode. Will need to Be in OCTAL mode
//! to access the register per the device driver.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_enter_halfsleep(void *pHandle);

//*****************************************************************************
//
//! @brief Function to Exit Half Sleep
//!
//! @param pHandle - Handle to the PSRAM.
//!
//! This function resets the device to bring it out of halfsleep
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_psram_aps25616n_exit_halfsleep(void *pHandle);

//
// This function runs approximately 200+ns on the 96MHz clock
//
// That results in the steps being 1us/32 ~= 31.25ns per step  (96MHz Clock)
//                                 1us/64 ~= 15.625ns per step (192MHz clock)
//


//*****************************************************************************
//!
//! @brief Delay Function to Exit Half Sleep
//!
//! This function runs approximately 200ns+ on the 96MHz clock
//!     60ns - 500ns is the range for this delay
//!
//! If running on the 192MHz clock, the user would need to modify delay to 2.
//! Since this is a fine delay, do not add the if statement to check clock here
//!
//*****************************************************************************

#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
extern __asm void APS25616N_tXPHS_delay( uint32_t ui32Iterations );
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
extern __attribute__((always_inline)) void APS25616N_tXPHS_delay( uint32_t ui32Iterations );
#elif defined(__GNUC_STDC_INLINE__)
extern void APS25616N_tXPHS_delay( uint32_t ui32Iterations );
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
extern __stackless inline void APS25616N_tXPHS_delay( uint32_t ui32Iterations );
#pragma diag_default = Pe940    // Restore IAR compiler warning
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_PSRAM_APS25616N_H
#endif

//*****************************************************************************
// End Doxygen group.
//! @}
//
//*****************************************************************************
