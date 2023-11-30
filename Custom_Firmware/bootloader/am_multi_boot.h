//*****************************************************************************
//
//! @file am_multi_boot.h
//!
//! @brief Prototype for Multi Protocol Bootloader implementation
//!
//! This is a bootloader program that supports flash programming over UART,
//! SPI, and I2C. The correct protocol is selected automatically at boot time.
//!
//! SWO is configured in 1M baud, 8-n-1 mode.
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

#ifndef AM_MULTI_BOOT_H
#define AM_MULTI_BOOT_H

#include "am_mcu_apollo.h"
#include "am_bootloader.h"

// All functions return 0 on success
typedef int (*flash_read_func_t)(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length);
typedef int (*flash_write_func_t)(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length);
typedef int (*flash_erase_func_t)(uint32_t ui32Addr);
typedef int (*flash_init_func_t)(void);
typedef int (*flash_deinit_func_t)(void);
typedef int (*flash_enable_func_t)(void);
typedef int (*flash_disable_func_t)(void);

typedef struct
{
    // Granularity for Write
    // Should be power of 2
    uint32_t                flashPageSize;
    // Granularity for Erase
    // Should be power of 2
    uint32_t                flashSectorSize;
    // Initialize the flash device
    flash_init_func_t       flash_init;
    // De-Initialize the flash device
    flash_deinit_func_t     flash_deinit;
    // Enable (Power up) the flash device
    flash_enable_func_t     flash_enable;
    // Disable (Put in Low power mode) the flash device
    flash_disable_func_t    flash_disable;
    // Read a block of data from within a flash page
    flash_read_func_t       flash_read_page;
    // Read a block of data within a flash page
    flash_write_func_t      flash_write_page;
    // Erase the flash sector corresponding to address specified
    flash_erase_func_t      flash_erase_sector;
} am_multiboot_flash_info_t;

#define OTA_INFO_OPTIONS_EXT_FLASH  0x1
#define OTA_INFO_OPTIONS_DATA       0x2
#define OTA_INFO_MAGIC_NUM          0xDEADCAFE
typedef struct
{
    // Should be set to OTA_INFO_MAGIC_NUM
    uint32_t    magicNum;
    // Address in flash where the new image should be programmed
    uint32_t    *pui32LinkAddress;
    // Length of image blob
    uint32_t    ui32NumBytes;
    // CRC of the image blob
    uint32_t    ui32ImageCrc;
    // (Optional) Security Info length
    uint32_t    secInfoLen;
    // Options - e.g. Read from external flash device
    uint32_t    ui32Options;
    // (optional) Security Information location
    uint32_t    *pui32SecInfoPtr;
    // Location of image blob - Address needs to be aligned to 4 Byte address
    uint32_t    *pui32ImageAddr;
    // CRC to confirm integrity of the OTA Descriptor structure
    uint32_t    ui32Crc;
} am_multiboot_ota_t;

typedef void (*invalidate_ota_func_t)(am_multiboot_ota_t *pOtaInfo);


// Internal flash info and handlers
extern am_multiboot_flash_info_t g_intFlash;

extern bool am_multiboot_init(uint32_t *pBuf, uint32_t bufSize);

extern void am_multiboot_uart_isr_handler(uint32_t ui32Module);
extern uint32_t am_multiboot_uart_detect_baudrate(uint32_t ui32RxPin);
extern void am_multiboot_setup_serial(int32_t i32Module, uint32_t ui32BaudRate);

extern void am_multiboot_ios_acc_isr_handler(void);
extern void am_multiboot_setup_ios_interface(uint32_t interruptPin);
extern void am_multiboot_cleanup_ios_interface(void);

// pExtFlash - can be NULL if using internal flash
// pTempBuf should point to a memory big enough to hold one flash sector (int or ext) worth of data
extern bool am_multiboot_ota_handler(am_multiboot_ota_t *pOtaInfo, uint32_t *pTempBuf,
                                     uint32_t tempBufSize, invalidate_ota_func_t invalidateOtaFunc,
                                     am_multiboot_flash_info_t *pExtFlash);

extern bool am_multiboot_check_boot_from_flash(bool *pbOverride, am_bootloader_image_t **ppsImage);
extern bool am_multiboot_get_main_image_info(uint32_t *pui32LinkAddr, uint32_t *pui32Length);

#endif // AM_MULTI_BOOT_H
