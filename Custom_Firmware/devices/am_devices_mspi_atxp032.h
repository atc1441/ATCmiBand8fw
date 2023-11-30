//*****************************************************************************
//
//! @file am_devices_mspi_atxp032.h
//!
//! @brief ATXP032 Multibit SPI Flash driver.
//!
//! @addtogroup atxp032 ATXP032 MSPI Driver
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

#ifndef AM_DEVICES_MSPI_ATXP032_H
#define AM_DEVICES_MSPI_ATXP032_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Global definitions for flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_ATXP032_WRITE_STATUS      0x01
#define AM_DEVICES_MSPI_ATXP032_PAGE_PROGRAM      0x02
#define AM_DEVICES_MSPI_ATXP032_READ              0x03
#define AM_DEVICES_MSPI_ATXP032_WRITE_DISABLE     0x04
#define AM_DEVICES_MSPI_ATXP032_READ_STATUS       0x65
#define AM_DEVICES_MSPI_ATXP032_READ_STATUS_BYTE1 0x05
#define AM_DEVICES_MSPI_ATXP032_WRITE_ENABLE      0x06
#define AM_DEVICES_MSPI_ATXP032_FAST_READ         0x0B
#define AM_DEVICES_MSPI_ATXP032_READ_4B           0x13
#define AM_DEVICES_MSPI_ATXP032_SUBSECTOR_ERASE   0x20
#define AM_DEVICES_MSPI_ATXP032_DUAL_READ         0x3B
#define AM_DEVICES_MSPI_ATXP032_DUAL_IO_READ      0xBB
#define AM_DEVICES_MSPI_ATXP032_WRITE_ENHVOL_CFG  0x61
#define AM_DEVICES_MSPI_ATXP032_RESET_ENABLE      0x66
#define AM_DEVICES_MSPI_ATXP032_QUAD_READ         0x6B
#define AM_DEVICES_MSPI_ATXP032_WRITE_VOL_CFG     0x81
#define AM_DEVICES_MSPI_ATXP032_RESET_MEMORY      0x99
#define AM_DEVICES_MSPI_ATXP032_READ_ID           0x9F
#define AM_DEVICES_MSPI_ATXP032_ENTER_4B          0xB7
#define AM_DEVICES_MSPI_ATXP032_BULK_ERASE        0xC7
#define AM_DEVICES_MSPI_ATXP032_SECTOR_ERASE      0xD8
#define AM_DEVICES_MSPI_ATXP032_EXIT_4B           0xE9
#define AM_DEVICES_MSPI_ATXP032_QUAD_IO_READ      0xEB
#define AM_DEVICES_MSPI_ATXP032_READ_QUAD_4B      0xEC
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_ATXP032_WEL         0x00000002        // Write enable latch
#define AM_DEVICES_MSPI_ATXP032_WIP         0x00000001        // Write in progress
//! @}

//*****************************************************************************
//
//! @name Device specific identification.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_ATXP032_ID        0x0043A700
#define AM_DEVICES_MSPI_ATXP032_ID_MASK   0x00FFFFFF
//! @}

//*****************************************************************************
//
//! @name Device specific definitions for flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_ATXP032_ENTER_QUAD_MODE      0x38
#define AM_DEVICES_ATXP032_UNPROTECT_SECTOR     0x39
#define AM_DEVICES_ATXP032_WRITE_STATUS_CTRL    0x71
#define AM_DEVICES_ATXP032_ENTER_OCTAL_MODE     0xE8
#define AM_DEVICES_ATXP032_ECHO_WITH_INVSERSION 0xA5
#define AM_DEVICES_ATXP032_RETURN_TO_SPI_MODE   0xFF
//! @}

//*****************************************************************************
//
//! @name Device specific definitions for the flash size information
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_ATXP032_PAGE_SIZE       0x100     //256 bytes, minimum program unit
#define AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE     0x10000   //64K bytes
//#define AM_DEVICES_MSPI_ATXP032_SECTOR_SIZE     0x1000   //4K bytes
#define AM_DEVICES_MSPI_ATXP032_MAX_BLOCKS      256
#define AM_DEVICES_MSPI_ATXP032_MAX_SECTORS     256      // Sectors within 4-byte address range.
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_ATXP032_RSTE        0x00000010        // Reset enable
#define AM_DEVICES_ATXP032_WEL         0x00000002        // Write enable latch
#define AM_DEVICES_ATXP032_WIP         0x00000001        // Operation in progress
//! @}

//*****************************************************************************
//
//! @name Global definitions for the MSPI instance to use.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_ATXP032_MSPI_INSTANCE     0

#define AM_DEVICES_MSPI_ATXP032_MAX_DEVICE_NUM    1
//! @}

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS,
    AM_DEVICES_MSPI_ATXP032_STATUS_ERROR
} am_devices_mspi_atxp032_status_t;

typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ScramblingStartAddr;
    uint32_t ui32ScramblingEndAddr;
} am_devices_mspi_atxp032_config_t;

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
typedef struct
{
    uint32_t ui32Turnaround;
    uint32_t ui32Rxneg;
    uint32_t ui32Rxdqsdelay;
} am_devices_mspi_atxp032_sdr_timing_config_t;
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
typedef struct
{
    bool            bTxNeg;
    bool            bRxNeg;
    bool            bRxCap;
    uint8_t         ui8TxDQSDelay;
    uint8_t         ui8RxDQSDelay;
    uint8_t         ui8Turnaround;
} am_devices_mspi_atxp032_sdr_timing_config_t;
#endif

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Initialize the mspi_flash driver.
//!
//! @param ui32Module    - Module number
//! @param psMSPISettings - Pointer to settings passed tp driver
//! @param ppHandle       - Pointer to driver handle pointer
//! @param ppMspiHandle   - Pointer to MSPI driver handle pointer
//!
//! @note This function should be called before any other am_devices_spiflash
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//!
//! @return status from am_devices_mspi_atxp032_status_t
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_init(uint32_t ui32Module,
                                           am_devices_mspi_atxp032_config_t *psMSPISettings,
                                           void **ppHandle, void **ppMspiHandle);

//*****************************************************************************
//
//! @brief De-Initialization the mspi_flash driver.
//!
//! @param pHandle - Pointer to driver handle
//!
//! @return status from am_devices_mspi_atxp032_status_t
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_deinit(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the ID of the external flash and returns the value.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function reads the device ID register of the external flash, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_id(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the current status of the external flash
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function reads the device ID register of the external flash, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_reset(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the current status of the external flash
//!
//! This function reads the status register of the external flash, and returns
//! the result as an 8-bit unsigned integer value. The processor will block
//! during the data transfer process, but will return as soon as the status
//! register had been read.
//!
//! Macro definitions for interpreting the contents of the status register are
//! included in the header file.
//!
//! @return 8-bit status register contents
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_status(void *pHandle, uint32_t *pStatus);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pHandle - Pointer to driver handle
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//! @param bWaitForCompletion - Wait for transfer to complete
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_read(void *pHandle, uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param pHandle - Pointer to driver handle
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//! @param bWaitForCompletion - When true, function will block for completion
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
extern uint32_t am_devices_mspi_atxp032_write(void *pHandle, uint8_t *pui8TxBuffer,
                                            uint32_t ui32WriteAddress,
                                            uint32_t ui32NumBytes,
                                            bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Erases the entire contents of the external flash
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function uses the "Bulk Erase" instruction to erase the entire
//! contents of the external flash.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_mass_erase(void *pHandle);

//*****************************************************************************
//
//! @brief Erases the contents of a single sector of flash
//!
//! @param pHandle - Pointer to driver handle
//! @param ui32SectorAddress - Address to erase in the external flash
//!
//! This function erases a single sector of the external flash as specified by
//! ui32EraseAddress. The entire sector where ui32EraseAddress will
//! be erased.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_sector_erase(void *pHandle, uint32_t ui32SectorAddress);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external FLASH into XIP mode.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function sets the external psram device and the MSPI into XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_enable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external FLASH from XIP mode.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function removes the external device and the MSPI from XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_disable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external FLASH into scrambling mode.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function sets the external psram device and the MSPI into scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_enable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external FLASH from scrambling mode.
//!
//! @param pHandle - Pointer to driver handle
//!
//! This function removes the external device and the MSPI from scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_disable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief
//!
//! @param pHandle
//! @param pui8RxBuffer
//! @param ui32ReadAddress
//! @param ui32NumBytes
//! @param ui32PauseCondition
//! @param ui32StatusSetClr
//! @param pfnCallback
//! @param pCallbackCtxt
//! @return
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_read_adv(void *pHandle,
                               uint8_t *pui8RxBuffer,
                               uint32_t ui32ReadAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pHandle - Pointer to driver handle
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//! @param bWaitForCompletion - Function will block when this is true
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_read_hiprio(void *pHandle,
                           uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
//*****************************************************************************
//
//! @brief Test signal integrity and delays on IO lines using echo with inversion command .
//!
//! @param pHandle - handle to the mspi instance.
//! @param pattern - data pattern to be used for the test.
//! @param length - data pattern to be used for the test.
//! @param pui8RxBuffer - handle to the mspi instance.
//!
//! This function reads the data of the echo with inversion command used for signal
//! integrity and delay on IO lines test.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_atxp032_echo_with_inversion(void *pHandle,
                                                uint8_t pattern,
                                                uint32_t length,
                                                uint8_t *pui8RxBuffer);
#endif

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
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
extern uint32_t am_devices_mspi_atxp032_sdr_init_timing_check(uint32_t module,
                                            am_devices_mspi_atxp032_config_t *pDevCfg,
                                            am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg);

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
extern uint32_t am_devices_mspi_atxp032_apply_sdr_timing(void *pHandle,
                       am_devices_mspi_atxp032_sdr_timing_config_t *pDevSdrCfg);

#endif


#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_ATXP032_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

