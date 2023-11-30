//*****************************************************************************
//
//! @file am_multi_boot_private.h
//!
//! @brief Internal definitions/structures shared within multiboot
//!
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

#ifndef AM_MULTI_BOOT_PRIVATE_H
#define AM_MULTI_BOOT_PRIVATE_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
// Include config before other bootloader files
#ifdef AM_MULTIBOOT_CONFIG_FILE
#include AM_MULTIBOOT_CONFIG_FILE
#endif

#include "am_bootloader.h"

#ifdef MULTIBOOT_SECURE
#include "am_multi_boot_secure.h"
#endif

//*****************************************************************************
//
// I2C Address to use
//
//*****************************************************************************
#ifndef I2C_SLAVE_ADDR
#define I2C_SLAVE_ADDR                     0x10
#endif

//*****************************************************************************
//
// Run without flag page.
//
//*****************************************************************************
#ifndef USE_FLAG_PAGE
#define USE_FLAG_PAGE                       0
#endif

//*****************************************************************************
//
// Location of the flag page.
//
//*****************************************************************************
#ifndef FLAG_PAGE_LOCATION
#define FLAG_PAGE_LOCATION                 0x00004000
#endif

//*****************************************************************************
//
// Max Size of Bootloader.
//
//*****************************************************************************
// The value here must match (at least) with the ROLength restriction imposed at
// bootloader linker configuration
#ifndef MAX_BOOTLOADER_SIZE
#define MAX_BOOTLOADER_SIZE                0x00004000
#endif
// The value here must match (at least) with the RWLength restriction imposed at
// bootloader linker configuration
#ifndef MAX_SRAM_USED
#define MAX_SRAM_USED                      0x00004000
#endif

extern am_bootloader_image_t *g_psBootImage;

//*****************************************************************************
//
// Safety Checks.
//
//*****************************************************************************
#if USE_FLAG_PAGE == 1
#if FLAG_PAGE_LOCATION & (AM_HAL_FLASH_PAGE_SIZE - 1)
#error "Flag Page address not page aligned"
#endif
#if FLAG_PAGE_LOCATION < MAX_BOOTLOADER_SIZE
#error "Flag Page overlaps with Bootloader"
#endif
#endif

//*****************************************************************************
//
// Default settings.
//
//*****************************************************************************
#ifndef DEFAULT_LINK_ADDRESS
#define DEFAULT_LINK_ADDRESS                ((uint32_t *) 0x00008000)
#endif
// Default override configured as invalid
#ifndef DEFAULT_OVERRIDE_GPIO
#define DEFAULT_OVERRIDE_GPIO               (0xFFFFFFFF)
#endif
#ifndef DEFAULT_OVERRIDE_POLARITY
#define DEFAULT_OVERRIDE_POLARITY           AM_BOOTLOADER_OVERRIDE_LOW
#endif

//*****************************************************************************
//
// Boot Loader Version Number
//
//*****************************************************************************
#define AM_BOOTLOADER_VERSION_NUM           0x00000001

//*****************************************************************************
//
// Boot messages.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK                   0x00000000
#define AM_BOOTLOADER_NAK                   0x00000001
#define AM_BOOTLOADER_READY                 0x00000002
#define AM_BOOTLOADER_IMAGE_COMPLETE        0x00000003
#define AM_BOOTLOADER_BAD_CRC               0x00000004
#define AM_BOOTLOADER_ERROR                 0x00000005
#define AM_BOOTLOADER_BL_VERSION            0x00000006
#define AM_BOOTLOADER_FW_VERSION            0x00000007

//*****************************************************************************
//
// Boot Commands.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK_CMD               0x00000000
#define AM_BOOTLOADER_NAK_CMD               0x00000001
#define AM_BOOTLOADER_NEW_IMAGE             0x00000002
#define AM_BOOTLOADER_NEW_PACKET            0x00000003
#define AM_BOOTLOADER_RESET                 0x00000004
#define AM_BOOTLOADER_SET_OVERRIDE_CMD      0x00000005
#define AM_BOOTLOADER_BL_VERSION_CMD        0x00000006
#define AM_BOOTLOADER_FW_VERSION_CMD        0x00000007
#define AM_BOOTLOADER_NEW_ENCRYPTED_IMAGE   0x00000008
#define AM_BOOTLOADER_RESTART               0x00000009

//*****************************************************************************
//
// Globals to keep track of the image write state.
//
//*****************************************************************************
extern uint32_t g_ui32BytesReceived;
extern uint32_t g_ui32CRC;

//*****************************************************************************
//
// Image structure to hold data about the downloaded boot image.
//
//*****************************************************************************
extern am_bootloader_image_t g_sImage;

//*****************************************************************************
//
// Function declarations.
//
//*****************************************************************************
extern bool
image_start_packet_read(am_bootloader_image_t *psImage, uint32_t *pui32Packet);
extern void
image_data_packet_read(uint8_t *pui8Src, uint32_t ui32Size);
extern void
program_image(uint32_t bEncrypted);

#ifdef MULTIBOOT_SECURE
extern void wipe_sram(void);
#endif

#endif // AM_MULTI_BOOT_PRIVATE_H
