//*****************************************************************************
//
//! @file am_multi_boot.c
//!
//! @brief Bootloader implementation accepting multiple host protocols.
//!
//! This is a bootloader implementation that supports flash programming over
//! UART, SPI, and I2C. The correct protocol is selected automatically at boot
//! time. The messaging is expected to follow little-endian format, which is
//! native to Apollo1/2.
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
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_multi_boot_private.h"
#include "am_multi_boot.h"

// Protection against NULL pointer
#define FLASH_OPERATE(pFlash, func) ((pFlash)->func ? (pFlash)->func() : 0)

//*****************************************************************************
//
// Message buffers.
//
// Note: The RX buffer needs to be 32-bit aligned to be compatible with the
// flash helper functions, but we also need an 8-bit pointer to it for copying
// data from the IOS interface, which is only 8 bits wide.
//
//*****************************************************************************
typedef struct
{
    uint32_t *pui32UserBuf;
    uint8_t  *pui8RxBuffer;
    uint32_t ui32BytesInBuffer;
    bool     bStoreInSRAM;
    uint32_t *pui32WriteAddress;
#ifdef MULTIBOOT_SECURE
    uint32_t ui32SramBytesUsed;
#endif
} am_multiboot_t;

static am_multiboot_t g_am_multiboot = {
    .pui32UserBuf = NULL,
    .pui8RxBuffer = NULL,
    .ui32BytesInBuffer = 0,
    .bStoreInSRAM = 0,
    .pui32WriteAddress = 0,
#ifdef MULTIBOOT_SECURE
    .ui32SramBytesUsed = 0,
#endif
} ;

static bool
check_flash_address_range(uint32_t address, uint32_t size);


//*****************************************************************************
//
// Globals to keep track of the image write state.
//
//*****************************************************************************
uint32_t g_ui32BytesReceived = 0;
uint32_t g_ui32CRC = 0;

//*****************************************************************************
//
// Image structure to hold data about the downloaded boot image.
//
//*****************************************************************************
am_bootloader_image_t g_sImage =
{
    DEFAULT_LINK_ADDRESS,
    0,
    0,
    DEFAULT_OVERRIDE_GPIO,
    DEFAULT_OVERRIDE_POLARITY,
    0,
    0,
    0
};

//*****************************************************************************
//
// Flag page information.
//
//*****************************************************************************
am_bootloader_image_t *g_psBootImage = (am_bootloader_image_t *) FLAG_PAGE_LOCATION;

// Checks that the address does not overlap with bootloader or flag page
// It also checks that the address is inside the internal flash
static bool
check_flash_address_range(uint32_t address, uint32_t size)
{
    static uint32_t g_intFlashSize = 0;
    am_hal_mcuctrl_device_t sDevice;

    uint32_t ui32Start = address;
    uint32_t ui32End = address + size - 1 ;

    if (g_intFlashSize == 0) // First call
    {
        // Get chip specific info
#if AM_APOLLO3_MCUCTRL
        am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);
#else // AM_APOLLO3_MCUCTRL
        am_hal_mcuctrl_device_info_get(&sDevice);
#endif // AM_APOLLO3_MCUCTRL

#if !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4L) &&!defined(AM_PART_APOLLO4P)
        g_intFlashSize = sDevice.ui32FlashSize;
#else
        g_intFlashSize = sDevice.ui32MRAMSize;
#endif
    }

    //
    // Make sure the address is within flash.
    //
    //
    // Check to make sure address is not within bootloader program
    //
    if ( ui32Start < MAX_BOOTLOADER_SIZE )
    {
        return false;
    }
    // Check to make sure the address is not beyond the flash
    if (ui32End >= g_intFlashSize)
    {
        return false;
    }
    if ( USE_FLAG_PAGE )
    {
        //
        // Check to make sure address is not in the flag page
        //
        if ( (FLAG_PAGE_LOCATION == ui32Start) ||
             ((FLAG_PAGE_LOCATION < ui32Start) &&
                ((FLAG_PAGE_LOCATION + AM_HAL_FLASH_PAGE_SIZE) > ui32Start)) ||
            ((FLAG_PAGE_LOCATION > ui32Start) &&
                (FLAG_PAGE_LOCATION <= ui32End))
           )
        {
            return false;
        }
    }
    return true;
}

//*****************************************************************************
//
// Internal Flash handler wrapper
//
//*****************************************************************************
static int
am_multiboot_flash_read_page(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length)
{
    if (check_flash_address_range((uint32_t)pSrc, ui32Length))
    {
        memcpy((uint8_t *)ui32DestAddr, (uint8_t *)pSrc, ui32Length);
        return 0;
    }
    else
    {
        return -1;
    }
}

static int
am_multiboot_flash_write_page(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length)
{
    if (check_flash_address_range(ui32DestAddr, ui32Length))
    {
        am_bootloader_program_flash_page(ui32DestAddr, pSrc, ui32Length);
        return 0;
    }
    else
    {
        return -1;
    }
}


static int am_multiboot_flash_erase_page(uint32_t ui32DestAddr)
{
    if (check_flash_address_range(ui32DestAddr, 4))
    {
        am_bootloader_erase_flash_page(ui32DestAddr);
        return 0;
    }
    else
    {
        return -1;
    }
}

am_multiboot_flash_info_t g_intFlash =
{
    // flagPageSize could be set as small as 4 Bytes, and as large as
    // AM_HAL_FLASH_PAGE_SIZE, since Read/Write on internal flash are
    // allowed at 4 byte granularity
    // It is set to max for power optimization
    // This may have indirect impact on temp storage needed, so it can
    // be reduced as a trade-off
    .flashPageSize      = AM_HAL_FLASH_PAGE_SIZE,
    .flashSectorSize    = AM_HAL_FLASH_PAGE_SIZE,
    .flash_init         = NULL,
    .flash_deinit       = NULL,
    .flash_enable       = NULL,
    .flash_disable      = NULL,
    .flash_read_page    = am_multiboot_flash_read_page,
    .flash_write_page   = am_multiboot_flash_write_page,
    .flash_erase_sector = am_multiboot_flash_erase_page,
};

am_multiboot_flash_info_t *g_pFlashInfo;
uint32_t *g_pTempBuf;

#ifdef MULTIBOOT_SECURE
// Wipe Clean SRAM up to the specified address
// CAUTION!!!
// This will wipe the complete SRAM including stack of the caller
// This should be called as the last thing before calling reset
void wipe_sram(void)
{
    //
    // Wipe SRAM (without using variables).
    //
    // Use the first SRAM location as temp
    // Last SRAM word = lastAddr = SRAM_BASEADDR + g_am_multiboot.ui32SramBytesUsed - 4;
    *((volatile uint32_t *)(SRAM_BASEADDR)) =
        SRAM_BASEADDR + g_am_multiboot.ui32SramBytesUsed - 4;

    // Can not use any local variables from now on
    while ( *((volatile uint32_t *)(SRAM_BASEADDR)) != SRAM_BASEADDR )
    {
        *(*((volatile uint32_t **)(SRAM_BASEADDR))) = 0x0;
        *((volatile uint32_t *)(SRAM_BASEADDR)) -= 4;
    }
}
#endif

// Programs the flash based on g_am_multiboot.pui32WriteAddress, g_am_multiboot.pui8RxBuffer & g_am_multiboot.ui32BytesInBuffer
void
program_image(uint32_t bEncrypted)
{
    uint32_t ui32WriteAddr = (uint32_t)g_am_multiboot.pui32WriteAddress;
    uint32_t *pui32ReadAddr = (uint32_t *)g_am_multiboot.pui8RxBuffer;
    uint32_t ui32NumBytes = g_am_multiboot.ui32BytesInBuffer;

    if ( g_am_multiboot.bStoreInSRAM )
    {
        while ( ui32NumBytes )
        {
            am_bootloader_program_flash_page(ui32WriteAddr, pui32ReadAddr,
                (ui32NumBytes > AM_HAL_FLASH_PAGE_SIZE) ? AM_HAL_FLASH_PAGE_SIZE: ui32NumBytes);
            if ( ui32NumBytes > AM_HAL_FLASH_PAGE_SIZE )
            {
                ui32NumBytes -= AM_HAL_FLASH_PAGE_SIZE;
                ui32WriteAddr += AM_HAL_FLASH_PAGE_SIZE;
                pui32ReadAddr += AM_HAL_FLASH_PAGE_SIZE / 4;
            }
            else
            {
                break;
            }
        }
    }
}

//*****************************************************************************
//
//! @brief Initialize multiboot
//!
//! @param pBuf is the temporary buffer for multiboot to operate on. This should
//! be at least equal to the AM_HAL_FLASH_PAGE_SIZE
//! @param bufSize is the temporary buffer size
//!
//! This function provides multiboot with the scratch memory in SRAM
//!
//! @return true if the parameters are acceptable.
//
//*****************************************************************************
bool
am_multiboot_init(uint32_t *pBuf, uint32_t bufSize)
{
    bool ret = false;
    if (pBuf && (bufSize >= AM_HAL_FLASH_PAGE_SIZE))
    {
        g_am_multiboot.pui32UserBuf = pBuf;
        ret = true;
    }
    return ret;
}

//*****************************************************************************
//
//! @brief Read an image start packet
//!
//! @param psImage is the image structure to read the packet into.
//!
//! This function reads the "new image" packet, and uses that
//! packet to fill in a bootloader image structure. The caller is responsible
//! for verifying the packet type before calling this function.
//! Packet Structure:
//! word0 = Link Address
//! word1 = Number of Bytes (Image Size)
//! word2 = CRC
//! ENCRYPTED?? (#ifdef MULTIBOOT_SECURE)
//! word3 = Security Trailer Length
//!
//! @return true if the image parameters are acceptable.
//
//*****************************************************************************
bool
image_start_packet_read(am_bootloader_image_t *psImage, uint32_t *pui32Packet)
{
    am_hal_mcuctrl_device_t sDevice;

    // Get chip specific info
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);
#else // AM_APOLLO3_MCUCTRL
    // Get chip specific info
    am_hal_mcuctrl_device_info_get(&sDevice);
#endif // AM_APOLLO3_MCUCTRL

    //
    // Set the image structure parameters based on the information in the
    // packet.
    //
    psImage->pui32LinkAddress = (uint32_t *)(pui32Packet[1]);
    psImage->ui32NumBytes = pui32Packet[2];
    psImage->ui32CRC = pui32Packet[3];
    psImage->ui32OverrideGPIO = DEFAULT_OVERRIDE_GPIO;
    psImage->ui32OverridePolarity = DEFAULT_OVERRIDE_POLARITY;
    psImage->bEncrypted = 0; // This only indicates Copy-Protection in flash

    //
    // We'll need to fill in the stack pointer and reset vector a little later
    // in the process.
    //
    psImage->pui32StackPointer = 0;
    psImage->pui32ResetVector = 0;

    //
    // Check to make sure we're not overwriting the bootloader or the flag page.
    //
    if (!check_flash_address_range((uint32_t)psImage->pui32LinkAddress,
            psImage->ui32NumBytes))
    {
        return false;
    }
    // Determine if we can gather image in SRAM completely before flashing all at once
    // This implementation uses the excess SRAM available in the system
    // CAUTION!!!: For this to work it is essential that the unused SRAM banks are
    // not powered down
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    if ((sDevice.ui32DTCMSize - MAX_SRAM_USED) >= psImage->ui32NumBytes)
#else
    if ((sDevice.ui32SRAMSize - MAX_SRAM_USED) >= psImage->ui32NumBytes)
#endif
    {
        g_am_multiboot.bStoreInSRAM = 1;
        g_am_multiboot.pui8RxBuffer = (uint8_t *)(SRAM_BASEADDR + MAX_SRAM_USED);
#ifdef MULTIBOOT_SECURE
        g_am_multiboot.ui32SramBytesUsed = sDevice.ui32SRAMSize;
#endif
    }
    else
    {
        g_am_multiboot.bStoreInSRAM = 0;
        if (g_am_multiboot.pui32UserBuf == NULL)
        {
            return false;
        }
        g_am_multiboot.pui8RxBuffer = (uint8_t *)g_am_multiboot.pui32UserBuf;
#ifdef MULTIBOOT_SECURE
        g_am_multiboot.ui32SramBytesUsed = MAX_SRAM_USED;
#endif
    }

#ifdef MULTIBOOT_SECURE
    // Validate the security trailer & Initialize the security params
    if ( init_multiboot_secure(pui32Packet[4], &pui32Packet[5], g_am_multiboot.bStoreInSRAM,
                               psImage, &psImage->bEncrypted) != 0 )
    {
        return false;
    }
#endif
    //
    // Otherwise, the image is presumed to be reasonable. Set our global
    // variables based on the new image structure.
    //
    g_am_multiboot.pui32WriteAddress = psImage->pui32LinkAddress;
    g_ui32BytesReceived = 0;
    g_am_multiboot.ui32BytesInBuffer = 0;
    g_ui32CRC = 0;
    return true;
}

//*****************************************************************************
//
//! @brief Read an image start packet
//!
//! @param psImage is the image structure to read the packet into.
//!
//! This function reads the "new image" packet, and uses that
//! packet to fill in a bootloader image structure. The caller is responsible
//! for verifying the packet type before calling this function.
//!
//! @return None.
//
//*****************************************************************************
void
image_data_packet_read(uint8_t *pui8Src, uint32_t ui32Size)
{
    uint32_t i;
    //
    // Loop through the data, copying it into the global buffer.
    //
    for ( i = 0; i < ui32Size; i++ )
    {
        g_am_multiboot.pui8RxBuffer[g_am_multiboot.ui32BytesInBuffer] = *pui8Src++;

        //
        // Keep track of how much data we've copied into the SRAM buffer.
        //
        g_am_multiboot.ui32BytesInBuffer++;
        g_ui32BytesReceived++;

        //
        // Whenever we hit a page boundary or the end of the image, we should
        // write to flash.
        //
        if ( (!g_am_multiboot.bStoreInSRAM && (g_am_multiboot.ui32BytesInBuffer == AM_HAL_FLASH_PAGE_SIZE)) ||
                 g_ui32BytesReceived == g_sImage.ui32NumBytes )
        {
            //
            // Run a quick CRC on the received bytes, holding on to the result in a
            // global variable, so we can pick up where we left off on the next pass.
            //
            am_bootloader_partial_crc32(g_am_multiboot.pui8RxBuffer, g_am_multiboot.ui32BytesInBuffer, &g_ui32CRC);

#ifdef MULTIBOOT_SECURE
            // Decrypt in place
            multiboot_secure_decrypt(g_am_multiboot.pui8RxBuffer, g_am_multiboot.ui32BytesInBuffer);
#endif

            //
            // If this is the first block of our new image, we need to record
            // the reset vector and stack pointer information for inclusion in
            // the flag page.
            //
            if ( g_am_multiboot.bStoreInSRAM || (g_ui32BytesReceived <= AM_HAL_FLASH_PAGE_SIZE) )
            {
                g_sImage.pui32StackPointer = (uint32_t *)(((uint32_t *)g_am_multiboot.pui8RxBuffer)[0]);
                g_sImage.pui32ResetVector = (uint32_t *)(((uint32_t *)g_am_multiboot.pui8RxBuffer)[1]);
            }

            if ( !g_am_multiboot.bStoreInSRAM )
            {
                am_bootloader_program_flash_page((uint32_t)g_am_multiboot.pui32WriteAddress,
                    (uint32_t *)g_am_multiboot.pui8RxBuffer, g_am_multiboot.ui32BytesInBuffer);
                //
                // Adjust the global variables.
                //
                g_am_multiboot.pui32WriteAddress += (g_am_multiboot.ui32BytesInBuffer / 4);
                g_am_multiboot.ui32BytesInBuffer = 0;
            }
        }
    }
}

//*****************************************************************************
//
//! @brief Check if we should be booting from flash with a valid image
//!
//! @param pbOverride is the return parameter, used to pass back the override status
//! @param ppsImage is the return parameter, used to pass back the image structure.
//!
//! This function checks the flag page (if enabled) and verifies the flash image
//! for integrity. It also checks for the override pin status in case forced
//! host boot is requested..
//!
//! @return true if it's okay to boot from flash (returns the image structure).
//
//*****************************************************************************
bool
am_multiboot_check_boot_from_flash(bool *pbOverride, am_bootloader_image_t **ppsImage)
{
    bool bValid = false;
    //
    // If we're using a flag page, we can run a full CRC check to verify the
    // integrity of our image. If not, we'll just check the override pin.
    // First check if the flag page is valid
    //
    if ( USE_FLAG_PAGE &&
        ( am_bootloader_validate_structure((uint32_t *)g_psBootImage, sizeof(*g_psBootImage)) ))
    {
        //
        // Check the flag page (including the stored CRC) and the override pin
        // to make sure we have a valid image and the host isn't requesting an
        // upgrade.
        //
        if (am_hal_bootloader_override_check(g_psBootImage))
        {
            *pbOverride = true;
        }
        else
        {
            *pbOverride = false;
            if ( am_bootloader_flash_check(g_psBootImage) )
            {
                *ppsImage = g_psBootImage;
                bValid = true;
            }
        }
    }
    else
    {
        //
        // Check the override pin to make sure the host isn't requesting an
        // upgrade, and do a quick check to make sure an image actually exists
        // at the default application location.
        //
        if (am_hal_bootloader_override_check(&g_sImage))
        {
            *pbOverride = true;
        }
        else
        {
            *pbOverride = false;
            if ( *(g_sImage.pui32LinkAddress) != 0xFFFFFFFF)
            {
                *ppsImage = &g_sImage;
                bValid = true;
            }
        }
    }
    return bValid;
}


// Erases the flash based on ui32Addr & ui32NumBytes
void
erase_ota_image(uint32_t ui32Addr, uint32_t ui32NumBytes, am_multiboot_flash_info_t *pFlash)
{
    // Erase the image
    while ( ui32NumBytes )
    {
        pFlash->flash_erase_sector(ui32Addr);
        if ( ui32NumBytes > pFlash->flashSectorSize )
        {
            ui32NumBytes -= pFlash->flashSectorSize;
            ui32Addr += pFlash->flashSectorSize;
        }
        else
        {
            break;
        }
    }
}

// Can write across pages
// The write address should be page aligned & the length in multiple of page size
int
write_to_flash(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length, am_multiboot_flash_info_t *pFlash)
{
    if (ui32DestAddr & (pFlash->flashPageSize - 1))
    {
        return -1 ;
    }
    while (ui32Length)
    {
        uint32_t ui32BytesInPage =
            (ui32Length > pFlash->flashPageSize) ? \
                pFlash->flashPageSize : ui32Length;
        // Writes are always page size
        pFlash->flash_write_page(ui32DestAddr, pSrc, pFlash->flashPageSize);
        pSrc += ui32BytesInPage / 4;
        ui32Length -= ui32BytesInPage;
        ui32DestAddr += ui32BytesInPage;
    }
    return 0;
}

// Can read across pages
int
read_from_flash(uint32_t ui32DestAddr, uint32_t *pSrc, uint32_t ui32Length, am_multiboot_flash_info_t *pFlash)
{
    uint32_t ui32Preceding = (uint32_t)pSrc & (pFlash->flashPageSize - 1);
    while (ui32Length)
    {
        uint32_t ui32BytesInPage =
            ((ui32Preceding + ui32Length) > pFlash->flashPageSize) ? \
                (pFlash->flashPageSize - ui32Preceding) : ui32Length;
        pFlash->flash_read_page(ui32DestAddr, pSrc, ui32BytesInPage);
        pSrc += ui32BytesInPage / 4;
        ui32Length -= ui32BytesInPage;
        ui32DestAddr += ui32BytesInPage;
        ui32Preceding = 0;
    }
    return 0;
}


// We can not read and program the same flash bank
// So - we need to first copy data to SRAM, and then flash...block by block
static void
program_image_from_flash(uint32_t ui32WriteAddr, uint32_t *pui32ReadAddr,
                         uint32_t ui32NumBytes, bool bDecrypt,
                         am_multiboot_flash_info_t *pReadFlash,
                         am_multiboot_flash_info_t *pWriteFlash)
{
    uint32_t ui32NumBytesInPage;
    uint32_t *pStart = g_pTempBuf;
    // Determine the preceding data bytes at the destination page
    uint32_t ui32PrecedingBytes = ui32WriteAddr & (pWriteFlash->flashSectorSize - 1);
    // Flash Write can only happen in terms of pages
    // So, if the image does not start on page boundary - need to take proper precautions
    // to preserve other data in the page
    if (ui32PrecedingBytes)
    {
        // Page aligned
        ui32WriteAddr &= ~(pWriteFlash->flashSectorSize - 1);
        // Copy the preceding content at destination page in buffer
        read_from_flash((uint32_t)g_pTempBuf, (uint32_t *)ui32WriteAddr, ui32PrecedingBytes, pWriteFlash);
    }
    while ( ui32NumBytes )
    {
        pStart = g_pTempBuf + ui32PrecedingBytes / 4;
        if ((ui32PrecedingBytes + ui32NumBytes) > pWriteFlash->flashSectorSize)
        {
            ui32NumBytesInPage = pWriteFlash->flashSectorSize - ui32PrecedingBytes;
        }
        else
        {
            // Last sector to be written
            ui32NumBytesInPage = ui32NumBytes;
            if ((ui32NumBytesInPage + ui32PrecedingBytes) != pWriteFlash->flashSectorSize)
            {
                // Copy the trailing content at destination page in buffer
                read_from_flash((uint32_t)pStart + ui32NumBytesInPage,
                                pui32ReadAddr + ui32NumBytesInPage / 4,
                                pWriteFlash->flashSectorSize - (ui32NumBytesInPage + ui32PrecedingBytes),
                                pWriteFlash);
            }
        }
        // Read the image data from source
        read_from_flash((uint32_t)pStart, pui32ReadAddr, ui32NumBytesInPage, pReadFlash);
#ifdef MULTIBOOT_SECURE
        if (bDecrypt)
        {
            // Decrypt in place
            multiboot_secure_decrypt(pStart, ui32NumBytesInPage);
        }
#endif
        // erase the sector
        pWriteFlash->flash_erase_sector(ui32WriteAddr);
        // Write the flash sector
        write_to_flash(ui32WriteAddr, g_pTempBuf, pWriteFlash->flashSectorSize, pWriteFlash);

        ui32WriteAddr += pWriteFlash->flashSectorSize;
        pui32ReadAddr += ui32NumBytesInPage / 4;
        ui32NumBytes -= ui32NumBytesInPage;
        ui32PrecedingBytes = 0;
    }
}

//*****************************************************************************
//
//! @brief Multiboot protocol handler for OTA update
//!
//! @param pOtaInfo is the pointer to OTA descriptor with image information
//! @param pTempBuf is the pointer to a temporary buffer, sized to one flash
//! page (bigger of internal and external flash page, if ext flash is being used)
//! @param invalidateOtaFunc is the function called to invalidate the OTA for
//! subsequent boots
//! @param pExtFlash is the pointer external flash access info if needed
//!
//! This function validates the OTA blob, and installs the image if verified.
//! It updates the flag page with the new image information and issues a POI
//!
//! @return false if OTA upgrade fails. Otherwise this function does not return
//
//*****************************************************************************
bool
am_multiboot_ota_handler(am_multiboot_ota_t *pOtaInfo, uint32_t *pTempBuf,
                         uint32_t tempBufSize, invalidate_ota_func_t invalidateOtaFunc,
                         am_multiboot_flash_info_t *pExtFlash)
{
    am_bootloader_image_t *psImage = &g_sImage;
    am_multiboot_flash_info_t *pFlash;

    if ((pTempBuf == NULL) || (pOtaInfo == NULL) || (pOtaInfo->magicNum != OTA_INFO_MAGIC_NUM))
    {
        return false;
    }

    // Validate the contents
    if ( !am_bootloader_validate_structure((uint32_t *)pOtaInfo, sizeof(*pOtaInfo)) )
    {
        return false;
    }

    //
    // Check to make sure we're not overwriting the bootloader or the flag page.
    //
    if (!check_flash_address_range((uint32_t)pOtaInfo->pui32LinkAddress,
        pOtaInfo->ui32NumBytes))
    {
        return false;
    }
    // Validate the ext flash info
    if (pOtaInfo->ui32Options & OTA_INFO_OPTIONS_EXT_FLASH)
    {
        if (pExtFlash && pExtFlash->flash_read_page &&
            pExtFlash->flash_write_page && pExtFlash->flash_erase_sector &&
            (pExtFlash->flashSectorSize <= tempBufSize))
        {
            pFlash = pExtFlash;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // Validate the address and the temp buf size
        if (g_intFlash.flashSectorSize > tempBufSize)
        {
            return false;
        }
        pFlash = &g_intFlash;
    }

    g_pTempBuf = pTempBuf;

    //
    // Set the image structure parameters based on the information in the
    // packet.
    //
    psImage->pui32LinkAddress = pOtaInfo->pui32LinkAddress;
    psImage->ui32NumBytes = pOtaInfo->ui32NumBytes;
    psImage->ui32CRC = pOtaInfo->ui32ImageCrc;
    psImage->ui32OverrideGPIO = DEFAULT_OVERRIDE_GPIO;
    psImage->ui32OverridePolarity = DEFAULT_OVERRIDE_POLARITY;
    psImage->bEncrypted = 0; // This only indicates Copy-Protection in flash

    //
    // We'll need to fill in the stack pointer and reset vector a little later
    // in the process.
    //
    psImage->pui32StackPointer = 0;
    psImage->pui32ResetVector = 0;

    g_am_multiboot.bStoreInSRAM = 0;

    g_am_multiboot.pui8RxBuffer = (uint8_t *)pOtaInfo->pui32ImageAddr;
    g_am_multiboot.ui32BytesInBuffer = pOtaInfo->ui32NumBytes;

    if (FLASH_OPERATE(pFlash, flash_init) == 0)
    {
        if (FLASH_OPERATE(pFlash, flash_enable) != 0)
        {
            FLASH_OPERATE(pFlash, flash_deinit);
        }
    }
    else
    {
        return false;
    }
#ifdef MULTIBOOT_SECURE
    g_am_multiboot.ui32SramBytesUsed = MAX_SRAM_USED;
    // Validate the security trailer & Initialize the security params
    // CAUTION: If the secInfo is in ext flash, it is assumed that the function would
    // have means to access it within
    // Ambiq OTA copies the secInfo always in internal flash, so that would not be
    // an issue
    if ( init_multiboot_secure(pOtaInfo->secInfoLen, pOtaInfo->pui32SecInfoPtr,
                1, psImage, &psImage->bEncrypted) != 0 )
    {
        FLASH_OPERATE(pFlash, flash_disable);
        FLASH_OPERATE(pFlash, flash_deinit);
        return false;
    }
    // Decrypt page by page
    program_image_from_flash((uint32_t)pOtaInfo->pui32ImageAddr, pOtaInfo->pui32ImageAddr,
        pOtaInfo->ui32NumBytes, true, pFlash, pFlash);
    // Verify
    if ( multiboot_secure_verify(&psImage->ui32CRC) )
    {
        // Erase the OTA image
        erase_ota_image((uint32_t)pOtaInfo->pui32ImageAddr, pOtaInfo->ui32NumBytes, pFlash);
        FLASH_OPERATE(pFlash, flash_disable);
        FLASH_OPERATE(pFlash, flash_deinit);
        return false;
    }
#endif
    psImage->pui32StackPointer = (uint32_t *)(((uint32_t *)pOtaInfo->pui32ImageAddr)[0]);
    psImage->pui32ResetVector = (uint32_t *)(((uint32_t *)pOtaInfo->pui32ImageAddr)[1]);

    //
    // The image is presumed to be reasonable. Set our global
    // variables based on the new image structure.
    //
    g_am_multiboot.pui32WriteAddress = psImage->pui32LinkAddress;

    program_image_from_flash((uint32_t)pOtaInfo->pui32LinkAddress, pOtaInfo->pui32ImageAddr,
        pOtaInfo->ui32NumBytes, false, pFlash, &g_intFlash);
    // Protect the image if needed
    program_image(psImage->bEncrypted);
    if ( !(pOtaInfo->ui32Options & OTA_INFO_OPTIONS_DATA) && USE_FLAG_PAGE )
    {
        //
        // Write the flag page.
        //
        am_bootloader_flag_page_update(&g_sImage, (uint32_t *)FLAG_PAGE_LOCATION);
    }
    if (invalidateOtaFunc)
    {
        invalidateOtaFunc(pOtaInfo);
    }
#ifdef MULTIBOOT_SECURE
    // Erase the OTA image
    // Special handling for the case when the install address overlaps with the OTA image
    // We need to add special handling so as to now erase part of freshly installed image
    if (((uint32_t)pOtaInfo->pui32LinkAddress < (uint32_t)pOtaInfo->pui32ImageAddr) &&
        ((uint32_t)pOtaInfo->pui32LinkAddress + pOtaInfo->ui32NumBytes > (uint32_t)pOtaInfo->pui32ImageAddr))
    {
        uint32_t skipBytes = (uint32_t)pOtaInfo->pui32LinkAddress + pOtaInfo->ui32NumBytes - (uint32_t)pOtaInfo->pui32ImageAddr;
        // Multiple of sector size
        skipBytes = (skipBytes + pFlash->flashSectorSize - 1) & ~(pFlash->flashSectorSize - 1);
        erase_ota_image((uint32_t)pOtaInfo->pui32ImageAddr + skipBytes, pOtaInfo->ui32NumBytes - skipBytes, pFlash);
    }
    else
    {
        erase_ota_image((uint32_t)g_am_multiboot.pui8RxBuffer, g_am_multiboot.ui32BytesInBuffer, pFlash);
    }
    wipe_sram();
#endif
    //
    // Perform a software reset.
    //
#if (defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P) || defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P))
    am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0);
#else
    am_hal_reset_poi();
#endif

    // Should never reach here
    return true;
}


//*****************************************************************************
//
//! @brief Get the information about the main image
//!
//! @param pui32LinkAddr - Used to return the link address for main image
//! @param pui32Length - Used to return the length of the image
//!
//! This function is used to determine attributes of the main image currently
//! in flash
//!
//! @return false if there is no valid flag page
//
//*****************************************************************************
bool
am_multiboot_get_main_image_info(uint32_t *pui32LinkAddr, uint32_t *pui32Length)
{
    bool bValid = false;
    //
    // If we're using a flag page, we can run a full CRC check to verify the
    // integrity of our image. If not, we'll just check the override pin.
    //
    if ( USE_FLAG_PAGE )
    {
        // First check if the flag page is valid
        if ( am_bootloader_validate_structure((uint32_t *)g_psBootImage, sizeof(*g_psBootImage)) )
        {
            *pui32LinkAddr = (uint32_t)g_psBootImage->pui32LinkAddress;
            *pui32Length = g_psBootImage->ui32NumBytes;
            bValid = true;
        }
    }
    return bValid;
}
