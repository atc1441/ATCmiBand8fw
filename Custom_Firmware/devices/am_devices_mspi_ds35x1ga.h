//*****************************************************************************
//
//! @file am_devices_mspi_ds35x1ga.h
//!
//! @brief Multibit SPI ds35x1ga NAND flash driver.
//!
//! @addtogroup mspi_ds35x1ga DX35X1GA MSPI Driver
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

#ifndef AM_DEVICES_MSPI_DS35X1GA_H
#define AM_DEVICES_MSPI_DS35X1GA_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name  Global definitions for flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X1 0x02
#define AM_DEVICES_MSPI_DS35X1GA_PROGRAM_LOAD_X4 0x32
#define AM_DEVICES_MSPI_DS35X1GA_PROGRAM_EXECUTE 0x10

#define AM_DEVICES_MSPI_DS35X1GA_READ_CELL_ARRAY 0x13
#define AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X1  0x03
#define AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X2  0x3B
#define AM_DEVICES_MSPI_DS35X1GA_READ_BUFFER_X4  0x6B

#define AM_DEVICES_MSPI_DS35X1GA_WRITE_DISABLE   0x04
#define AM_DEVICES_MSPI_DS35X1GA_WRITE_ENABLE    0x06

#define AM_DEVICES_MSPI_DS35X1GA_READ_ID         0x9F

#define AM_DEVICES_MSPI_DS35X1GA_BLOCK_ERASE     0xD8

#define AM_DEVICES_MSPI_DS35X1GA_SET_FEATURE     0x1F
#define AM_DEVICES_MSPI_DS35X1GA_GET_FEATURE     0x0F
#define AM_DEVICES_MSPI_DS35X1GA_FEATURE_STATUS  0xC0
#define AM_DEVICES_MSPI_DS35X1GA_FEATURE_B0      0xB0
#define AM_DEVICES_MSPI_DS35X1GA_FEATURE_A0      0xA0

#define AM_DEVICES_MSPI_DS35X1GA_RESET           0xFF
//! @}

//*****************************************************************************
//
//! @name  Device specific identification.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_DS35X1GA_ID      0xE500
#define AM_DEVICES_MSPI_DS35X1GA_ID_MASK 0x00FF00
//! @}
//*****************************************************************************
//
//! @name  Device specific definitions for the flash size information
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_DS35X1GA_PAGE_DATA_SIZE 2048
#define AM_DEVICES_MSPI_DS35X1GA_PAGE_OOB_SIZE  64
#define AM_DEVICES_MSPI_DS35X1GA_PAGE_FULL_SIZE 2112 //Internal ECC is enabled, default; 64 bytes are used for redundancy or for other uses
#define AM_DEVICES_MSPI_DS35X1GA_BLOCK_SIZE     0x20000  //128K bytes
#define AM_DEVICES_MSPI_DS35X1GA_MAX_BLOCKS     1024
#define AM_DEVICES_MSPI_DS35X1GA_MAX_PAGES      65536
//! @}
//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_DS35X1GA_ECCS  0x30  // ECC Status[1:0]
#define AM_DEVICES_DS35X1GA_PRG_F 0x08 // Program Fail
#define AM_DEVICES_DS35X1GA_ERS_F 0x04 // Erase Fail
#define AM_DEVICES_DS35X1GA_WEL   0x02   // Write enable latch
#define AM_DEVICES_DS35X1GA_OIP   0x01   // Operation in progress

#define AM_DEVICES_DS35X1GA_ECCS_NO_BIT_FLIPS            0x00
#define AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED     0x10
#define AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_NOT_CORRECTED 0x20
#define AM_DEVICES_DS35X1GA_ECCS_BIT_FLIPS_CORRECTED_THR 0x30 // more than the threshold bit
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash OTP register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_DS35X1GA_OTP_PRT    0x80
#define AM_DEVICES_DS35X1GA_OTP_EN     0x40
#define AM_DEVICES_DS35X1GA_OTP_ECC_EN 0x10
#define AM_DEVICES_DS35X1GA_OTP_QC_EN  0x01
//! @}


//*****************************************************************************
//
//! @name Global definitions for the MSPI instance to use.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_DS35X1GA_MSPI_INSTANCE  0
#define AM_DEVICES_MSPI_DS35X1GA_MAX_DEVICE_NUM 1
//! @}
//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
//
//!
//
typedef enum
{
    AM_DEVICES_MSPI_DS35X1GA_STATUS_SUCCESS,
    AM_DEVICES_MSPI_DS35X1GA_STATUS_ERROR
} am_devices_mspi_ds35x1ga_status_t;

//
//!
//
typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ScramblingStartAddr;
    uint32_t ui32ScramblingEndAddr;
} am_devices_mspi_ds35x1ga_config_t;

typedef struct
{
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    uint32_t ui32Turnaround;
    uint32_t ui32Rxneg;
    uint32_t ui32Rxdqsdelay;
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
    bool            bTxNeg;
    bool            bRxNeg;
    bool            bRxCap;
    uint8_t         ui8TxDQSDelay;
    uint8_t         ui8RxDQSDelay;
    uint8_t         ui8Turnaround;
#endif
} am_devices_mspi_ds35x1ga_sdr_timing_config_t;
typedef enum
{
    AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_NO_BIT_FLIPS = 0,
    AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_CORRECTED,
    AM_DEVICES_MSPI_DS35X1GA_ECC_STATUS_BIT_FLIPS_NOT_CORRECTED
} am_devices_mspi_ds35x1ga_ecc_status_t;
//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
// ****************************************************************************
//
//! @brief
//!
//! @param ui32Module
//! @param psMSPISettings
//! @param ppHandle
//! @param ppMspiHandle
//!
//! @return
//
// ****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_init(uint32_t ui32Module,
                                                const am_devices_mspi_ds35x1ga_config_t *psMSPISettings,
                                                void **ppHandle, void **ppMspiHandle);

// ****************************************************************************
//
//! @brief
//!
//! @param pHandle
//!
//! @return
//
// ****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_deinit(void *pHandle);

//*****************************************************************************
//
//! @brief Read the ID of the NAND flash.
//!
//! @param pHandle - Ds35x1ga device handle.
//! @param pui32DeviceID - a variable pointer to store the ID of the NAND flash.
//!
//! This function reads ID of the external flash.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_id(void *pHandle, uint32_t *pui32DeviceID);


//*****************************************************************************
//
//! @brief Read the contents of a certain page from the NAND flash into a buffer.
//!
//! @param pHandle - Ds35x1ga device handle.
//! @param ui32PageNum - Page number, for this NAND, valid value from 0 to 65535.
//! @param pui8DataBuffer - Buffer to store the data read from the NAND data region.
//! @param ui32DataLen - Number of bytes to read from the NAND data region.
//! @param pui8OobBuffer - Buffer to store the data read from the NAND oob region.
//! @param ui32OobLen - Number of bytes to read from the NAND oob region.
//! @param pui32EccResult - Return ECC result according ECCS[1:0] value.
//! 0--no bit flips; 1--bit flips corrected; 2--bit flips not corrected.
//!
//! This function reads the external flash at the certain page and stores
//! the received data into the provided buffer location. This function will
//! only store ui32DataLen bytes of data region and ui32OobLen bytes of oob region.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_read(void *pHandle, uint32_t ui32PageNum,
                                                uint8_t *pui8DataBuffer,
                                                uint32_t ui32DataLen,
                                                uint8_t *pui8OobBuffer,
                                                uint32_t ui32OobLen,
                                                uint8_t *pui32EccResult);


//*****************************************************************************
//
//! @brief Write the contents of a certain page to the NAND flash.
//!
//! @param pHandle - Ds35x1ga device handle.
//! @param ui32PageNum - Page number, for this NAND, valid value from 0 to 65535.
//! @param pui8DataBuffer - Buffer to store the data writen to the NAND data region.
//! @param ui32DataLen - Number of bytes to write to the NAND data region.
//! @param pui8OobBuffer - Buffer to store the data write to the NAND oob region.
//! @param ui32OobLen - Number of bytes to write to the NAND oob region.
//!
//! This function writes the data to the certain page on the external flash.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_write(void *pHandle, uint32_t ui32PageNum,
                                                uint8_t *pui8DataBuffer,
                                                uint32_t ui32DataLen,
                                                uint8_t *pui8OobBuffer,
                                                uint32_t ui32OobLen);

//*****************************************************************************
//
//! @brief Erase a certain page of the NAND flash.
//!
//! @param pHandle - Ds35x1ga device handle.
//! @param ui32BlockNum - block number, for this NAND, valid value from 0 to 1024.
//!
//! This function erases a certain block of the external flash.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_ds35x1ga_block_erase(void *pHandle, uint32_t ui32BlockNum);


//*****************************************************************************
//
//! @brief Apply given SDR timing settings to target MSPI instance.
//!
//! @param pHandle - Handle to the NandFlash.
//! @param pDevSdrCfg - Pointer to the ddr timing config structure
//!
//! This function applies the ddr timing settings to the selected mspi instance.
//! This function must be called after MSPI instance is initialized into
//! ENABLEFINEDELAY0 = 1 mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t
am_devices_mspi_ds35x1ga_apply_sdr_timing(void *pHandle,
                                         am_devices_mspi_ds35x1ga_sdr_timing_config_t *pDevSdrCfg);

//*****************************************************************************
//
//! @brief Checks DS35X1GA timing and determine a delay setting.
//!
//! @param module
//! @param pDevCfg
//! @param pDevSdrCfg
//!
//! This function scans through the delay settings of MSPI SDR mode and selects
//! the best parameter to use by tuning TURNAROUND/RXNEG/RXDQSDELAY0 values.
//! This function is only valid in SDR mode and ENABLEDQS0 = 0.
//!
//! @return 32-bit status, scan result in structure type
//
//*****************************************************************************
extern uint32_t
am_devices_mspi_ds35x1ga_sdr_init_timing_check(uint32_t module,
                                              const am_devices_mspi_ds35x1ga_config_t *pDevCfg,
                                              am_devices_mspi_ds35x1ga_sdr_timing_config_t *pDevSdrCfg);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_DS35X1GA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
